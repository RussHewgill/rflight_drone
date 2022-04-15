pub struct Buffer<'a, T: 'a> {
    buffer: &'a mut [T],
    read_index: usize,
    write_index: usize,
}

impl<'a, T> Buffer<'a, T>
where
    T: Copy,
{
    pub fn new(buffer: &'a mut [T]) -> Buffer<T> {
        Buffer::<T> {
            buffer,
            read_index: 0,
            write_index: 0,
        }
    }

    pub fn size(&self) -> usize {
        if self.write_index >= self.read_index {
            self.write_index - self.read_index
        } else {
            self.write_index + self.buffer.len() - self.read_index
        }
    }

    pub fn next_contiguous_slice_len(&self) -> usize {
        if self.read_index == 0 {
            self.buffer.len() - self.write_index - 1
        } else if self.write_index >= self.read_index {
            self.buffer.len() - self.write_index
        } else {
            self.read_index - self.write_index - 1
        }
    }

    pub fn next_mut_slice(&mut self, n: usize) -> &mut [T] {
        if n > self.next_contiguous_slice_len() {
            panic!(
                "Not enough contiguous data to write into (wanted {}, have {})",
                n,
                self.next_contiguous_slice_len()
            );
        }

        let start = self.write_index;
        self.write_index = (self.write_index + n) % self.buffer.len();
        &mut self.buffer[start..start + n]
    }

    pub fn available_len(&self) -> usize {
        if self.read_index <= self.write_index {
            self.read_index + self.buffer.len() - self.write_index - 1
        } else {
            self.read_index - self.write_index - 1
        }
    }

    pub fn peek(&self, n: usize) -> T {
        if n >= self.size() {
            panic!("Peek out of range: {} requested, max {}", n, self.size());
        }
        self.buffer[(self.read_index + n) % self.buffer.len()]
    }

    pub fn take_slice(&mut self, n: usize, buf: &mut [T]) {
        if n > self.size() {
            panic!(
                "Not enough data to read (wanted {}, have {})",
                n,
                self.size()
            );
        }
        for (i, byte) in buf.iter_mut().enumerate().take(n) {
            *byte = self.buffer[(self.read_index + i) % self.buffer.len()];
        }
        self.read_index = (self.read_index + n) % self.buffer.len();
    }
}
