package com.example.isegyeboard.room_history

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.login.RoomApi
import kotlinx.coroutines.launch

class HistoryViewModel(private val roomLogId: String) : ViewModel() {
    private val HisrotyNetwork = BaseApi.getInstance().create(RoomApi::class.java)

    private val _historyList = MutableLiveData<List<OrderMenuResponse>>()
    val historyList: LiveData<List<OrderMenuResponse>> = _historyList

    fun getHistoryList() {
        viewModelScope.launch {
            try {
                val result = HisrotyNetwork.getHistoryList(roomLogId)
                println("result: $result")
                _historyList.value = result
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}
