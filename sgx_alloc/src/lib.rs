// Copyright (C) 2017-2018 Baidu, Inc. All Rights Reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//  * Neither the name of Baidu, Inc., nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! # liballoc crate for Rust SGX SDK
//!
//! This crate equals to the `liballoc_system` crate in Rust.
//! It connects Rust memory allocation to Intel SGX's sgx_tstd library.
//! It is essential, because we depends on Intel SGX's SDK.

#![no_std]

#![feature(global_allocator)]
#![feature(allocator_api)]
#![feature(alloc)]

extern crate sgx_trts;

extern crate alloc;
use self::alloc::heap::{Alloc, AllocErr, Layout, Excess, CannotReallocInPlace, Opaque};
use core::ptr::NonNull;

// The minimum alignment guaranteed by the architecture. This value is used to
// add fast paths for low alignment values. In practice, the alignment is a
// constant at the call site and the branch will be optimized out.
#[cfg(target_arch = "x86")]
const MIN_ALIGN: usize = 8;
#[cfg(target_arch = "x86_64")]
const MIN_ALIGN: usize = 16;

pub struct System;

unsafe impl Alloc for System {
    #[inline]
    unsafe fn alloc(&mut self, layout: Layout) -> Result<NonNull<Opaque>, AllocErr> {
        (&*self).alloc(layout)
    }

    #[inline]
    unsafe fn alloc_zeroed(&mut self, layout: Layout)
        -> Result<NonNull<Opaque>, AllocErr>
    {
        (&*self).alloc_zeroed(layout)
    }

    #[inline]
    unsafe fn dealloc(&mut self, ptr: NonNull<Opaque>, layout: Layout) {
        (&*self).dealloc(ptr, layout)
    }

    #[inline]
    unsafe fn realloc(&mut self,
                      ptr: NonNull<Opaque>,
                      layout: Layout,
                      new_size: usize) -> Result<NonNull<Opaque>, AllocErr> {
        (&*self).realloc(ptr, layout, new_size)
    }

    #[inline]
    fn usable_size(&self, layout: &Layout) -> (usize, usize) {
        (&self).usable_size(layout)
    }

    #[inline]
    unsafe fn alloc_excess(&mut self, layout: Layout) -> Result<Excess, AllocErr> {
        (&*self).alloc_excess(layout)
    }

    #[inline]
    unsafe fn realloc_excess(&mut self,
                             ptr: NonNull<Opaque>,
                             layout: Layout,
                             new_size: usize) -> Result<Excess, AllocErr> {
        (&*self).realloc_excess(ptr, layout, new_size)
    }

    #[inline]
    unsafe fn grow_in_place(&mut self,
                            ptr: NonNull<Opaque>,
                            layout: Layout,
                            new_size: usize) -> Result<(), CannotReallocInPlace> {
        (&*self).grow_in_place(ptr, layout, new_size)
    }

    #[inline]
    unsafe fn shrink_in_place(&mut self,
                              ptr: NonNull<Opaque>,
                              layout: Layout,
                              new_size: usize) -> Result<(), CannotReallocInPlace> {
        (&*self).shrink_in_place(ptr, layout, new_size)
    }
}

mod platform {

    use sgx_trts::libc::{self, c_void};
    use core::ptr;
    use core::ptr::NonNull;

    use MIN_ALIGN;
    use System;
    use alloc::heap::{Alloc, AllocErr, Layout, Opaque};

    unsafe impl<'a> Alloc for &'a System {
        #[inline]
        unsafe fn alloc(&mut self, layout: Layout) -> Result<NonNull<Opaque>, AllocErr> {
            let ptr = if layout.align() <= MIN_ALIGN && layout.align() <= layout.size() {
                libc::malloc(layout.size()) as *mut u8
            } else {
                aligned_malloc(&layout)
            };
            NonNull::new(ptr as *mut Opaque).ok_or(AllocErr)
        }

        #[inline]
        unsafe fn alloc_zeroed(&mut self, layout: Layout)
            -> Result<NonNull<Opaque>, AllocErr>
        {
            if layout.align() <= MIN_ALIGN && layout.align() <= layout.size() {
                let ptr = libc::calloc(layout.size(), 1) as *mut u8;
                NonNull::new(ptr as *mut Opaque).ok_or(AllocErr)
            } else {
                let ret = self.alloc(layout.clone());
                if let Ok(ptr) = ret {
                    ptr::write_bytes(ptr.as_ptr() as *mut c_void, 0, layout.size());
                }
                ret
            }
        }

        #[inline]
        unsafe fn dealloc(&mut self, ptr: NonNull<Opaque>, _layout: Layout) {
            libc::free(ptr.as_ptr() as *mut c_void)
        }
    }

    #[inline]
    unsafe fn aligned_malloc(layout: &Layout) -> *mut u8 {
        let mut out = ptr::null_mut();
        let ret = libc::posix_memalign(&mut out, layout.align(), layout.size());
        if ret != 0 {
            ptr::null_mut()
        } else {
            out as *mut u8
        }
    }
}
