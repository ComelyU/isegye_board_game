package com.example.presentation

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.viewModelScope
import com.example.domain.usecase.OrderUseCase
import com.example.domain.usecase.TurtleBotUseCase
import com.example.presentation.base.BaseViewModel
import com.example.presentation.ui.OrderDetailState
import com.example.presentation.ui.OrderUiState
import com.example.presentation.ui.UiState
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class MainViewModel @Inject constructor(
    private val turtleBotUseCase: TurtleBotUseCase,
    private val orderUseCase: OrderUseCase,
) :BaseViewModel<Unit>() {
    private val _uiStateLiveData = MutableLiveData<UiState>()
    val uiStateFlow: LiveData<UiState>
        get() = _uiStateLiveData

    init {
        _uiStateLiveData.value = UiState(0, 0, emptyList())
    }

    fun pushTurtle() {

    }

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
//            println("뷰모델 들어옴 ${response}")
            if (response.isSuccess) {
                val orderDataList = response.getOrThrow()
                _uiStateLiveData.value = _uiStateLiveData.value?.copy(
                    orders = orderDataList.map { orderData ->
                        OrderUiState(
                            orderId = orderData.orderId,
                            customerId = orderData.customerId,
                            orderStatus = orderData.orderStatus,
                            orderDetail = orderData.orderDetail.map { it ->
                                OrderDetailState(
                                    orderDetailId = it.orderDetailId,
                                    menuName = it.menuName,
                                    quantity = it.quantity,
                                    totalPrice = it.totalPrice
                                )
                            }
                        )
                    }
                )
            }
        }
    }
}