package com.example.isegyeboard.game_list

import android.content.Context
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.isegyeboard.baseapi.BaseApi
import kotlinx.coroutines.launch

class GameViewModel() : ViewModel() {
    private val gameNetwork = BaseApi.getInstance().create(GameApi::class.java)

    private val _gameList = MutableLiveData<List<GameClass>>()
    val gameList: LiveData<List<GameClass>> = _gameList

    fun getCurrentGameList(storeId: String) {
        println("게임 뷰모델 들어옴")
        viewModelScope.launch {
            try {
                val result = gameNetwork.getGameList(storeId)
                println(result)
                _gameList.value = result
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}