package com.example.isegyeboard.room_history

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.room_history.model.OrderGameResponse
import com.example.isegyeboard.room_history.model.OrderMenuResponse
import kotlinx.coroutines.launch

class HistoryViewModel(private val customerId: String) : ViewModel() {
    private val historyNetwork = BaseApi.getInstance().create(HistoryApi::class.java)


    private val _menuHistoryList = MutableLiveData<List<OrderMenuResponse>>()
    val menuHistoryList: LiveData<List<OrderMenuResponse>> = _menuHistoryList

    fun getMenuHistoryList() {
        viewModelScope.launch {
            try {
                val result = historyNetwork.getMenuHistoryList(customerId)
                println("result: $result")
                _menuHistoryList.value = result
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }


    private val _gameHistoryList = MutableLiveData<List<OrderGameResponse>>()
    val gameHistoryList: LiveData<List<OrderGameResponse>> = _gameHistoryList

    fun getGameHistoryList() {
        viewModelScope.launch {
            try {
                val result = historyNetwork.getGameHistoryList(customerId)
                println("result: $result")
                _gameHistoryList.value = result.orderGameList
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}
