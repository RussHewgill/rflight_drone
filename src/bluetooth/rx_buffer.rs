use arrayvec::ArrayVec;

pub struct Buffer<'a, T: 'a> {
    buffer:      &'a mut [T],
    read_index:  usize,
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

pub struct Buffer2<T, const SIZE: usize = 512> {
    // vec:         ArrayVec<T, SIZE>,
    vec:         [T; SIZE],
    read_index:  usize,
    write_index: usize,
}

impl<const SIZE: usize> Buffer2<u8, SIZE> {
    pub fn new() -> Buffer2<u8, SIZE> {
        Self {
            vec:         [0u8; SIZE],
            read_index:  0,
            write_index: 0,
        }
    }
}

impl<T: Copy, const SIZE: usize> Buffer2<T, SIZE> {
    // pub const fn len(&self) -> usize {
    //     self.vec.len()
    // }

    pub fn size(&self) -> usize {
        if self.write_index >= self.read_index {
            self.write_index - self.read_index
        } else {
            self.write_index + self.vec.len() - self.read_index
        }
    }

    pub fn next_contiguous_slice_len(&self) -> usize {
        if self.read_index == 0 {
            self.vec.len() - self.write_index - 1
        } else if self.write_index >= self.read_index {
            self.vec.len() - self.write_index
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
        self.write_index = (self.write_index + n) % self.vec.len();
        &mut self.vec[start..start + n]
    }

    pub fn peek(&self, n: usize) -> T {
        if n >= self.size() {
            panic!("Peek out of range: {} requested, max {}", n, self.size());
        }
        self.vec[(self.read_index + n) % self.vec.len()]
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
            *byte = self.vec[(self.read_index + i) % self.vec.len()];
        }
        self.read_index = (self.read_index + n) % self.vec.len();
    }
}
