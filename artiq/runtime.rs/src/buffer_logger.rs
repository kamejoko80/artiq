use core::cell::RefCell;
use log::{self, Log, LogMetadata, LogRecord, LogLevelFilter};
use log_buffer::LogBuffer;

pub struct BufferLogger {
    buffer: RefCell<LogBuffer<&'static mut [u8]>>
}

// We can never preempt from within the logger, so there can be no data races.
unsafe impl Sync for BufferLogger {}

impl BufferLogger {
    pub fn new(buffer: &'static mut [u8]) -> BufferLogger {
        BufferLogger {
            buffer: RefCell::new(LogBuffer::new(buffer))
        }
    }

    pub fn register<F: FnOnce(&BufferLogger)>(&self, f: F) {
        // log::set_logger_raw captures a pointer to ourselves, so we must prevent
        // ourselves from being moved or dropped after that function is called (and
        // before log::shutdown_logger_raw is called).
        unsafe {
            log::set_logger_raw(|max_log_level| {
                max_log_level.set(LogLevelFilter::Trace);
                self as *const Log
            }).expect("global logger can only be initialized once");
        }
        f(self);
        log::shutdown_logger_raw().unwrap();
    }

    pub fn clear(&self) {
        self.buffer.borrow_mut().clear()
    }

    pub fn extract<R, F: FnOnce(&str) -> R>(&self, f: F) -> R {
        f(self.buffer.borrow_mut().extract())
    }
}

impl Log for BufferLogger {
    fn enabled(&self, _metadata: &LogMetadata) -> bool {
        true
    }

    fn log(&self, record: &LogRecord) {
        if self.enabled(record.metadata()) {
            use core::fmt::Write;
            writeln!(self.buffer.borrow_mut(), "{:>5}({}): {}",
                     record.level(), record.location().module_path(), record.args()).unwrap();
            println!("{:>5}({}): {}",
                     record.level(), record.location().module_path(), record.args());
        }
    }
}
