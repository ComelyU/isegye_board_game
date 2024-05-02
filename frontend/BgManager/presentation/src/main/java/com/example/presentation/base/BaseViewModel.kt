package com.example.presentation.base

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.launch
import java.util.concurrent.atomic.AtomicBoolean

abstract class BaseViewModel<E>(
    private val defaultBlockTime: Long = 500L,
) : ViewModel() {

    private val _eventFlow = MutableSharedFlow<E>()
    val eventFlow = _eventFlow.asSharedFlow()

    private val availableTimeMap = mutableMapOf<Any, Long>()
    private val runningMap = mutableMapOf<Any, AtomicBoolean>()

    private var debounceJobMap = mutableMapOf<Any, Job>()
    protected fun triggerEvent(event: E) {
        viewModelScope.launch {
            _eventFlow.emit(event)
        }
    }

    protected fun throttle(key:Any = Unit, blockingTime: Long = defaultBlockTime, block: suspend() -> Unit) {
        val currentTime = System.currentTimeMillis()
        val isAvailable = availableTimeMap[key]?.let { it <= currentTime} ?: true
        if (!isAvailable) return
        val isRunning = runningMap.getOrPut(key) { AtomicBoolean() }

        val nextAvailableTime = currentTime + blockingTime
        availableTimeMap[key] = nextAvailableTime
        viewModelScope.launch {
            block()
            isRunning.set(false)
        }
    }

    protected fun debounce(key: Any = Unit, blockingTime: Long = defaultBlockTime, block: suspend () -> Unit) {
        debounceJobMap[key]?.cancel()
        debounceJobMap[key] = viewModelScope.launch {
            delay(blockingTime)
            block()
        }
    }
}