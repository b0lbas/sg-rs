#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]

use nix::sys::uio;
use std::ffi::c_void;
use std::fs::{File, OpenOptions};
use std::io::{self, IoSlice};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::path::Path;
use std::time::Duration;
#[cfg(feature = "polling")]
use {
    mio::event::Evented,
    mio::unix::EventedFd,
    mio::{Poll, PollOpt, Ready, Token},
};

pub mod sys {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
    pub const SG_FLAG_Q_AT_TAIL: u32 = 0x10;
}

#[derive(Debug, Copy, Clone)]
pub enum Direction {
    None,
    ToDevice,
    FromDevice,
    ToFromDevice,
}

impl Direction {
    fn to_underlying(self) -> std::os::raw::c_int {
        match self {
            Direction::None => sys::SG_DXFER_NONE,
            Direction::ToDevice => sys::SG_DXFER_TO_DEV,
            Direction::FromDevice => sys::SG_DXFER_FROM_DEV,
            Direction::ToFromDevice => sys::SG_DXFER_TO_FROM_DEV,
        }
    }
}

#[derive(Debug, Default)]
pub struct Task {
    inner: sys::sg_io_hdr,
    cmd: Vec<u8>,
    data: Vec<u8>,
    sense: Vec<u8>,
}

unsafe impl Send for Task {}
unsafe impl Sync for Task {}

impl Task {
    pub fn new() -> Self {
        Task {
            inner: sys::sg_io_hdr {
                interface_id: 'S' as std::os::raw::c_int,
                dxfer_direction: sys::SG_DXFER_NONE,
                ..Default::default()
            },
            cmd: Vec::new(),
            data: Vec::new(),
            sense: Vec::new(),
        }
    }

    fn sync_pointers(&mut self) {
        self.inner.cmdp = if self.cmd.is_empty() {
            std::ptr::null_mut()
        } else {
            self.cmd.as_mut_ptr()
        };
        self.inner.dxferp = if self.data.is_empty() {
            std::ptr::null_mut()
        } else {
            self.data.as_mut_ptr() as *mut c_void
        };
        self.inner.sbp = if self.sense.is_empty() {
            std::ptr::null_mut()
        } else {
            self.sense.as_mut_ptr()
        };
    }

    pub fn set_cdb(&mut self, buf: &[u8]) -> &mut Self {
        self.cmd = buf.to_vec();
        self.inner.cmd_len = buf.len() as u8;
        self.inner.cmdp = self.cmd.as_mut_ptr();
        self
    }

    pub fn cdb(&self) -> &[u8] {
        &self.cmd
    }

    pub fn set_timeout(&mut self, timeout: Duration) -> &mut Self {
        self.inner.timeout = timeout.as_millis() as u32;
        self
    }

    pub fn timeout(&self) -> Duration {
        Duration::from_millis(self.inner.timeout.into())
    }

    pub fn set_data(&mut self, buf: &[u8], direction: Direction) -> &mut Self {
        self.data = buf.to_vec();
        self.inner.dxferp = self.data.as_mut_ptr() as *mut c_void;
        self.inner.dxfer_len = buf.len() as u32;
        self.inner.dxfer_direction = direction.to_underlying();
        self
    }

    pub fn data(&self) -> &[u8] {
        &self.data
    }

    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }

    pub fn set_sense_buffer(&mut self, len: usize) -> &mut Self {
        self.sense = vec![0u8; len];
        self.inner.sbp = self.sense.as_mut_ptr();
        self.inner.mx_sb_len = len as u8;
        self
    }

    pub fn sense_buffer(&self) -> &[u8] {
        let valid = self.inner.sb_len_wr as usize;
        &self.sense[..valid.min(self.sense.len())]
    }

    pub fn set_flags(&mut self, flags: u32) -> &mut Self {
        self.inner.flags = flags;
        self
    }

    pub fn flags(&self) -> u32 {
        self.inner.flags
    }

    pub fn duration(&self) -> u32 {
        self.inner.duration
    }

    pub fn residual_data(&self) -> i32 {
        self.inner.resid
    }

    pub fn status(&self) -> u8 {
        self.inner.status
    }

    pub fn host_status(&self) -> u16 {
        self.inner.host_status
    }

    pub fn driver_status(&self) -> u16 {
        self.inner.driver_status
    }

    pub fn ok(&self) -> bool {
        (self.inner.info & sys::SG_INFO_OK_MASK) == sys::SG_INFO_OK
    }

    // usr_ptr requires a generic type parameter on Task to be sound; left unimplemented
}

impl Clone for Task {
    // derive(Clone) would copy inner verbatim, leaving cmdp/dxferp/sbp pointing
    // into the original's Vec allocations; sync_pointers fixes them after the copy
    fn clone(&self) -> Self {
        let mut t = Task {
            inner: self.inner,
            cmd: self.cmd.clone(),
            data: self.data.clone(),
            sense: self.sense.clone(),
        };
        t.sync_pointers();
        t.inner.cmd_len = self.inner.cmd_len;
        t.inner.dxfer_len = self.inner.dxfer_len;
        t.inner.mx_sb_len = self.inner.mx_sb_len;
        t.inner.dxfer_direction = self.inner.dxfer_direction;
        t
    }
}

pub struct Device(File);

impl Device {
    pub fn open<P: AsRef<Path>>(path: P) -> io::Result<Device> {
        Ok(Device(
            OpenOptions::new()
                .read(true)
                .write(true)
                .custom_flags(libc::O_NONBLOCK)
                .open(path)?,
        ))
    }

    /// Returns the number of tasks successfully sent.
    pub fn send(&self, tasks: &[Task]) -> io::Result<usize> {
        if tasks.is_empty() {
            return Ok(0);
        }

        // NOTE: sg_io_hdr is a struct, it is transmuted to an array/slice
        let iovecs: Vec<IoSlice> = tasks
            .iter()
            .map(|task| {
                io::IoSlice::new(unsafe {
                    std::slice::from_raw_parts(
                        &task.inner as *const sys::sg_io_hdr as *const u8,
                        std::mem::size_of::<sys::sg_io_hdr>(),
                    )
                })
            })
            .collect();

        let hdr_size = std::mem::size_of::<sys::sg_io_hdr>();
        let expected = tasks.len() * hdr_size;

        // partial writes handled: loop resumes from the next unsent header
        let mut written = 0usize;
        while written < expected {
            let offset = written / hdr_size;
            match uio::writev(&self.0, &iovecs[offset..]) {
                Ok(n) => written += n,
                Err(nix::errno::Errno::EINTR) => {}
                Err(e) => return Err(e.into()),
            }
        }

        Ok(written / hdr_size)
    }

    /// Returns the number of tasks received - how many were added to `tasks`.
    pub fn receive(&self, tasks: &mut Vec<Task>) -> io::Result<usize> {
        let mut hdrs = vec![sys::sg_io_hdr::default(); sys::SG_MAX_QUEUE as usize];
        let hdr_size = std::mem::size_of::<sys::sg_io_hdr>();

        let mut iovecs: Vec<io::IoSliceMut> = hdrs
            .iter_mut()
            .map(|hdr| {
                io::IoSliceMut::new(unsafe {
                    std::slice::from_raw_parts_mut(
                        hdr as *mut sys::sg_io_hdr as *mut u8,
                        hdr_size,
                    )
                })
            })
            .collect();

        let bytes_read = loop {
            match uio::readv(&self.0, iovecs.as_mut_slice()) {
                Ok(n) if n > 0 => break n,
                Ok(_) => return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "sg read returned 0")),
                Err(nix::errno::Errno::EINTR) => {}
                Err(nix::errno::Errno::EAGAIN) => {
                    // EAGAIN returns WouldBlock; caller decides whether to poll or sleep
                    return Err(io::Error::new(io::ErrorKind::WouldBlock, "no tasks available"))
                }
                Err(e) => return Err(e.into()),
            }
        };

        let tasks_read = bytes_read / hdr_size;

        for hdr in hdrs.into_iter().take(tasks_read) {
            let mut task = Task::new();

            // pointers in hdr refer to the caller's original buffers; copy data out
            // before hdr ownership moves into task.inner, then redirect via sync_pointers
            let cmd_len = hdr.cmd_len as usize;
            if !hdr.cmdp.is_null() && cmd_len > 0 {
                task.cmd = unsafe { std::slice::from_raw_parts(hdr.cmdp, cmd_len) }.to_vec();
            }

            let data_len = hdr.dxfer_len as usize;
            if !hdr.dxferp.is_null() && data_len > 0 {
                task.data =
                    unsafe { std::slice::from_raw_parts(hdr.dxferp as *const u8, data_len) }
                        .to_vec();
            }

            let sense_len = hdr.mx_sb_len as usize;
            if !hdr.sbp.is_null() && sense_len > 0 {
                task.sense =
                    unsafe { std::slice::from_raw_parts(hdr.sbp, sense_len) }.to_vec();
            }

            task.inner = hdr;
            task.sync_pointers();
            tasks.push(task);
        }

        Ok(tasks_read)
    }

    pub fn perform(&self, task: &mut Task) -> io::Result<()> {
        // sync before ioctl: set_data/set_cdb may have reallocated the Vec
        task.sync_pointers();

        #[cfg(target_env = "musl")]
        let request = sys::SG_IO as i32;
        #[cfg(not(target_env = "musl"))]
        let request: u64 = sys::SG_IO.into();

        let ret = unsafe { libc::ioctl(self.0.as_raw_fd(), request, &mut task.inner) };
        if ret == -1 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
}

impl AsRawFd for Device {
    fn as_raw_fd(&self) -> std::os::unix::io::RawFd {
        self.0.as_raw_fd()
    }
}

#[cfg(feature = "polling")]
impl Evented for Device {
    fn register(&self, poll: &Poll, token: Token, interest: Ready, opts: PollOpt) -> io::Result<()> {
        EventedFd(&self.as_raw_fd()).register(poll, token, interest, opts)
    }

    fn reregister(&self, poll: &Poll, token: Token, interest: Ready, opts: PollOpt) -> io::Result<()> {
        EventedFd(&self.as_raw_fd()).reregister(poll, token, interest, opts)
    }

    fn deregister(&self, poll: &Poll) -> io::Result<()> {
        EventedFd(&self.as_raw_fd()).deregister(poll)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_sys() {
        assert_eq!(super::sys::SG_IO, 0x2285);
    }

    #[test]
    fn test_cdb() {
        let cdb = [0x12u8; 6];
        let mut task = Task::new();
        task.set_cdb(&cdb);
        assert_eq!(task.cdb(), &cdb);
        assert_eq!(task.inner.cmd_len as usize, cdb.len());
        assert_eq!(task.inner.cmdp, task.cmd.as_ptr() as *mut u8);
    }

    #[test]
    fn test_clone_pointers() {
        let mut task = Task::new();
        task.set_cdb(&[0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00]);
        task.set_data(&[0u8; 512], Direction::FromDevice);
        task.set_sense_buffer(32);

        let cloned = task.clone();

        assert_eq!(cloned.inner.cmdp, cloned.cmd.as_ptr() as *mut u8);
        assert_eq!(cloned.inner.dxferp, cloned.data.as_ptr() as *mut c_void);
        assert_eq!(cloned.inner.sbp, cloned.sense.as_ptr() as *mut u8);

        assert_ne!(cloned.inner.cmdp, task.inner.cmdp);
        assert_ne!(cloned.inner.dxferp, task.inner.dxferp);
    }

    #[test]
    fn test_sense_buffer_written() {
        let mut task = Task::new();
        task.set_sense_buffer(32);
        task.inner.sb_len_wr = 18;
        assert_eq!(task.sense_buffer().len(), 18);
    }

    #[test]
    fn test_new_default_direction() {
        let task = Task::new();
        assert_eq!(task.inner.dxfer_direction, sys::SG_DXFER_NONE);
        assert_eq!(task.inner.interface_id, 'S' as std::os::raw::c_int);
    }
}
