#include <pico/mutex.h>
#include <sys/lock.h>

extern "C" {

void __attribute__((weak)) __retarget_lock_init(_LOCK_T lock) {
    mutex_init((mutex_t *)lock);
}

void __attribute__((weak)) __retarget_lock_init_recursive(_LOCK_T lock) {
    recursive_mutex_init((recursive_mutex_t *)lock);
}

void __attribute__((weak)) __retarget_lock_close(_LOCK_T lock) {
    (void)lock;
}

void __attribute__((weak)) __retarget_lock_close_recursive(_LOCK_T lock) {
    (void)lock;
}

void __attribute__((weak)) __retarget_lock_acquire(_LOCK_T lock) {
    mutex_enter_blocking((mutex_t *)lock);
}

void __attribute__((weak)) __retarget_lock_acquire_recursive(_LOCK_T lock) {
    recursive_mutex_enter_blocking((recursive_mutex_t *)lock);
}

int __attribute__((weak)) __retarget_lock_try_acquire(_LOCK_T lock) {
    return mutex_try_enter((mutex_t *)lock, nullptr);
}

int __attribute__((weak)) __retarget_lock_try_acquire_recursive(_LOCK_T lock) {
    return recursive_mutex_try_enter((recursive_mutex_t *)lock, nullptr);
}

void __attribute__((weak)) __retarget_lock_release(_LOCK_T lock) {
    mutex_exit((mutex_t *)lock);
}

void __attribute__((weak)) __retarget_lock_release_recursive(_LOCK_T lock) {
    recursive_mutex_exit((recursive_mutex_t *)lock);
}

}