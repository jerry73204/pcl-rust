//! Raw pointer wrapper for PCL objects with explicit ownership management
//!
//! This module provides safe abstractions over raw pointers to PCL objects,
//! respecting PCL's custom memory alignment requirements that are incompatible
//! with cxx::UniquePtr's default deleters.

use std::marker::PhantomData;
use std::ptr::NonNull;

/// A wrapper around raw pointers to PCL objects that manages ownership
/// and ensures proper cleanup using PCL's aligned deallocation.
///
/// # Safety
///
/// This type maintains safety through:
/// - Non-null pointer guarantee via NonNull<T>
/// - Explicit ownership semantics - exactly one PclPtr owns each object
/// - Proper cleanup via PCL's aligned delete functions
/// - No Clone implementation to prevent double-free
pub struct PclPtr<T: PclDrop> {
    ptr: NonNull<T>,
    _phantom: PhantomData<T>,
}

impl<T: PclDrop> PclPtr<T> {
    /// Creates a new PclPtr from a raw pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure:
    /// - The pointer is valid and points to a properly allocated PCL object
    /// - The pointer was allocated using PCL's aligned allocation
    /// - This PclPtr becomes the sole owner of the object
    /// - The pointer is not null
    #[inline]
    pub unsafe fn from_raw(ptr: *mut T) -> Option<Self> {
        NonNull::new(ptr).map(|ptr| Self {
            ptr,
            _phantom: PhantomData,
        })
    }

    /// Creates a new PclPtr from a NonNull pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure:
    /// - The pointer points to a properly allocated PCL object
    /// - The pointer was allocated using PCL's aligned allocation
    /// - This PclPtr becomes the sole owner of the object
    #[inline]
    pub unsafe fn from_non_null(ptr: NonNull<T>) -> Self {
        Self {
            ptr,
            _phantom: PhantomData,
        }
    }

    /// Returns a raw pointer to the object.
    ///
    /// The pointer is guaranteed to be non-null and valid as long as
    /// this PclPtr is alive.
    #[inline]
    pub fn as_ptr(&self) -> *const T {
        self.ptr.as_ptr()
    }

    /// Returns a mutable raw pointer to the object.
    ///
    /// The pointer is guaranteed to be non-null and valid as long as
    /// this PclPtr is alive.
    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.ptr.as_ptr()
    }

    /// Returns a shared reference to the object.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the lifetime of the returned reference
    /// does not exceed the lifetime of this PclPtr.
    #[inline]
    pub unsafe fn as_ref(&self) -> &T {
        unsafe { self.ptr.as_ref() }
    }

    /// Returns a mutable reference to the object.
    ///
    /// # Safety
    ///
    /// The caller must ensure that:
    /// - The lifetime of the returned reference does not exceed the lifetime of this PclPtr
    /// - No other references to the object exist during the lifetime of the returned reference
    #[inline]
    pub unsafe fn as_mut(&mut self) -> &mut T {
        unsafe { self.ptr.as_mut() }
    }

    /// Consumes the PclPtr and returns the raw pointer without calling the destructor.
    ///
    /// This transfers ownership back to the caller, who becomes responsible
    /// for proper cleanup.
    #[inline]
    pub fn into_raw(self) -> *mut T {
        let ptr = self.ptr.as_ptr();
        std::mem::forget(self);
        ptr
    }

    /// Returns the raw pointer without transferring ownership.
    ///
    /// Unlike `into_raw`, this does not consume the PclPtr.
    #[inline]
    pub fn get(&self) -> *mut T {
        self.ptr.as_ptr()
    }
}

// PclPtr is not Clone to prevent double-free errors
// PclPtr is not Copy for the same reason

// Send and Sync implementation depends on the underlying type T
unsafe impl<T: PclDrop + Send> Send for PclPtr<T> {}
unsafe impl<T: PclDrop + Sync> Sync for PclPtr<T> {}

impl<T: PclDrop> std::fmt::Debug for PclPtr<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PclPtr").field("ptr", &self.ptr).finish()
    }
}

impl<T: PclDrop> std::fmt::Pointer for PclPtr<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Pointer::fmt(&self.ptr.as_ptr(), f)
    }
}

/// Trait for types that can transfer ownership to/from raw pointers
/// while respecting PCL's memory management requirements.
pub trait TransferOwnership<T> {
    /// Transfers ownership from a raw pointer to a safe wrapper.
    ///
    /// # Safety
    ///
    /// The implementation must ensure:
    /// - The pointer is valid and properly allocated
    /// - The wrapper becomes the sole owner
    /// - Proper cleanup will occur when the wrapper is dropped
    unsafe fn take_ownership(ptr: *mut T) -> Option<Self>
    where
        Self: Sized;

    /// Transfers ownership from the wrapper to a raw pointer.
    ///
    /// The caller becomes responsible for proper cleanup.
    fn release_ownership(self) -> *mut T;

    /// Returns a raw pointer without transferring ownership.
    fn as_raw_ptr(&self) -> *const T;

    /// Returns a mutable raw pointer without transferring ownership.
    fn as_raw_mut_ptr(&mut self) -> *mut T;
}

impl<T: PclDrop> TransferOwnership<T> for PclPtr<T> {
    #[inline]
    unsafe fn take_ownership(ptr: *mut T) -> Option<Self> {
        unsafe { Self::from_raw(ptr) }
    }

    #[inline]
    fn release_ownership(self) -> *mut T {
        self.into_raw()
    }

    #[inline]
    fn as_raw_ptr(&self) -> *const T {
        self.as_ptr()
    }

    #[inline]
    fn as_raw_mut_ptr(&mut self) -> *mut T {
        self.as_mut_ptr()
    }
}

/// Trait for types that have explicit PCL-compatible drop behavior.
///
/// This trait is implemented for PCL types that require special
/// cleanup using PCL's aligned deallocation functions.
pub trait PclDrop {
    /// Explicitly drop the object using PCL's aligned deallocation.
    ///
    /// # Safety
    ///
    /// This function must only be called once per object, and the
    /// object must have been allocated using PCL's aligned allocation.
    unsafe fn pcl_drop(ptr: *mut Self);
}

/// Macro to implement PclDrop for PCL types.
///
/// This generates the appropriate FFI call to the C++ deletion function.
#[macro_export]
macro_rules! impl_pcl_drop {
    ($rust_type:ty, $delete_fn:ident) => {
        impl $crate::ptr::PclDrop for $rust_type {
            #[inline]
            unsafe fn pcl_drop(ptr: *mut Self) {
                if !ptr.is_null() {
                    $crate::ffi::$delete_fn(ptr);
                }
            }
        }
    };
}

// Implement Drop for PclPtr<T> where T: PclDrop
impl<T: PclDrop> Drop for PclPtr<T> {
    fn drop(&mut self) {
        unsafe {
            T::pcl_drop(self.ptr.as_ptr());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    // Mock type for testing
    struct MockPclType {
        data: i32,
    }

    static MOCK_DELETED: AtomicBool = AtomicBool::new(false);

    impl PclDrop for MockPclType {
        unsafe fn pcl_drop(ptr: *mut Self) {
            if !ptr.is_null() {
                // Simulate PCL's aligned delete
                MOCK_DELETED.store(true, Ordering::SeqCst);
                // In real implementation, this would call the FFI delete function
                unsafe {
                    drop(Box::from_raw(ptr));
                }
            }
        }
    }

    #[test]
    fn test_pcl_ptr_creation_and_drop() {
        MOCK_DELETED.store(false, Ordering::SeqCst);

        let obj = Box::into_raw(Box::new(MockPclType { data: 42 }));

        {
            let ptr = unsafe { PclPtr::from_raw(obj).unwrap() };
            assert_eq!(unsafe { ptr.as_ref().data }, 42);
        } // ptr should be dropped here

        assert!(
            MOCK_DELETED.load(Ordering::SeqCst),
            "Object should have been deleted"
        );
    }

    #[test]
    fn test_pcl_ptr_into_raw() {
        MOCK_DELETED.store(false, Ordering::SeqCst);

        let obj = Box::into_raw(Box::new(MockPclType { data: 42 }));

        let ptr = unsafe { PclPtr::from_raw(obj).unwrap() };
        let raw = ptr.into_raw(); // Should not call drop

        assert!(
            !MOCK_DELETED.load(Ordering::SeqCst),
            "Object should not be deleted yet"
        );

        // Manual cleanup
        unsafe {
            MockPclType::pcl_drop(raw);
        }

        assert!(
            MOCK_DELETED.load(Ordering::SeqCst),
            "Object should now be deleted"
        );
    }

    #[test]
    fn test_transfer_ownership_trait() {
        MOCK_DELETED.store(false, Ordering::SeqCst);

        let obj = Box::into_raw(Box::new(MockPclType { data: 42 }));

        let ptr = unsafe { PclPtr::<MockPclType>::take_ownership(obj).unwrap() };
        assert_eq!(ptr.as_raw_ptr(), obj);

        let raw = ptr.release_ownership();
        assert_eq!(raw, obj);
        assert!(
            !MOCK_DELETED.load(Ordering::SeqCst),
            "Object should not be deleted yet"
        );

        // Manual cleanup
        unsafe {
            MockPclType::pcl_drop(raw);
        }

        assert!(
            MOCK_DELETED.load(Ordering::SeqCst),
            "Object should now be deleted"
        );
    }

    #[test]
    fn test_null_pointer_handling() {
        let ptr = unsafe { PclPtr::<MockPclType>::from_raw(std::ptr::null_mut()) };
        assert!(ptr.is_none(), "Should return None for null pointer");
    }
}
