package com.example.presentation

import androidx.lifecycle.viewModelScope
import com.example.domain.usecase.OrderUseCase
import com.example.domain.usecase.TurtleBotUseCase
import com.example.presentation.base.BaseViewModel
import com.example.presentation.ui.OrderUiState
import com.example.presentation.ui.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class MainViewModel @Inject constructor(
    private val turtleBotUseCase: TurtleBotUseCase,
    private val orderUseCase: OrderUseCase
) :BaseViewModel<Unit>() {
    private val _uiStateFlow = MutableStateFlow(
        UiState(
            turtleId = 0,
            storeId = 0,
            orders = emptyList()
        )
    )
    val uiStateFlow = _uiStateFlow.asStateFlow()

    fun click() {
        viewModelScope.launch {
            val response = turtleBotUseCase.invoke()
            if (response.isSuccess) {
                val data = response.getOrThrow()
                _uiStateFlow.update { uiState ->
                    uiState.copy(
                        turtleId = data.id,
                        storeId = data.storeId
                    )
                }
            }
        }
    }

    fun loadData() {
        viewModelScope.launch {

            // Order 데이터 로드
            val response = orderUseCase.invoke()
            if (response.isSuccess) {
                val orderDataList = response.getOrThrow()
                val currentState = _uiStateFlow.value

                // Order 데이터 업데이트
                _uiStateFlow.value = currentState.copy(
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