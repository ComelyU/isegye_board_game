package com.example.bgmanager

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.launch

class OrderListViewModel() : ViewModel()  {
    private val manageNetwork = OrderNetwork()

    private val _orderList = MutableLiveData<List<OrderClass>>()
    val orderList: LiveData<List<OrderClass>> = _orderList

    fun getCurrentOrderList() {
        viewModelScope.launch {
            try {
                val result = manageNetwork.getOrderList()
//                println("뷰모델 들어옴 : $result")
                _orderList.value = result
            } catch (e: Exception) {
                // 에러 처리
                println("뷰모델 에러남 $e")
            }
        }
    }
}