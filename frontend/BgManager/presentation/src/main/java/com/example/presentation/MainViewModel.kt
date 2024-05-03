package com.example.presentation

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.viewModelScope
import com.example.domain.usecase.OrderUseCase
import com.example.domain.usecase.TurtleBotUseCase
import com.example.presentation.base.BaseViewModel
import com.example.presentation.ui.OrderUiState
import com.example.presentation.ui.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class MainViewModel @Inject constructor(
    private val turtleBotUseCase: TurtleBotUseCase,
    private val orderUseCase: OrderUseCase
) :BaseViewModel<Unit>() {
//) : ViewModel() {
//    private val _uiStateFlow = MutableStateFlow(
//        UiState(
//            turtleId = 0,
//            storeId = 0,
//            orders = emptyList()
//        )
//    )
    private val _uiStateLiveData = MutableLiveData<UiState>()
    val uiStateFlow: LiveData<UiState>
        get() = _uiStateLiveData

//    val uiStateFlow = _uiStateFlow.asStateFlow()

    init {
        _uiStateLiveData.value = UiState(0, 0, emptyList())
    }

//    fun callTurtle() {
//        viewModelScope.launch {
//            val response = turtleBotUseCase.invoke()
//            if (response.isSuccess) {
//                val data = response.getOrThrow()
//                _uiStateFlow.update { uiState ->
//                    uiState.copy(
//                        turtleId = data.id,
//                        storeId = data.storeId
//                    )
//                }
//            }
//        }
//    }

    fun pushTurtle() {

    }

//    fun loadData() {
//        viewModelScope.launch {
//
//            // Order 데이터 로드
//            val response = orderUseCase.invoke()
////            println(response)
//            if (response.isSuccess) {
//                val orderDataList = response.getOrThrow()
//                val currentState = _uiStateFlow.value
//
//                // Order 데이터 업데이트
//                _uiStateFlow.value = currentState.copy(
//                    orders = orderDataList.map { orderData ->
//                        OrderUiState(
//                            orderId = orderData.id,
//                            orderQuantity = orderData.quantity,
//                            orderName = orderData.orderName
//                        )
//                    }
//                )
//                println("뷰모델 데이터 ${_uiStateFlow.value}")
//            }
//        }
//    }

    fun callTurtle() {
        viewModelScope.launch {
            val response = turtleBotUseCase.invoke()
            if (response.isSuccess) {
                val data = response.getOrThrow()
                _uiStateLiveData.value = _uiStateLiveData.value?.copy(
                    turtleId = data.id,
                    storeId = data.storeId
                )
            }
        }
    }

    fun loadData() {
        viewModelScope.launch {
            val response = orderUseCase.invoke()
            if (response.isSuccess) {
                val orderDataList = response.getOrThrow()
                _uiStateLiveData.value = _uiStateLiveData.value?.copy(
                    orders = orderDataList.map { orderData ->
                        OrderUiState(
                            orderId = orderData.id,
                            orderQuantity = orderData.quantity,
                            orderName = orderData.orderName
                        )
                    }
                )
            }
        }
    }
}