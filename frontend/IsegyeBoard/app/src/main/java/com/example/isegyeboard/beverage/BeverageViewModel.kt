package com.example.isegyeboard.beverage

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.beverage.cart.CartClass
import com.example.isegyeboard.game_list.GameClass
import kotlinx.coroutines.launch
class BeverageViewModel : ViewModel() {
    private val beverageNetwork = BaseApi.getInstance().create(BeverageApi::class.java)

    private val _menuList = MutableLiveData<List<BeverageClass>>()
    val menuList: LiveData<List<BeverageClass>> = _menuList

    fun getMenuList(storeId: String) {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList(storeId)
                val allMenu = result
                println(result)
                _menuList.value = allMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }


    private val _cartItems = MutableLiveData<List<CartClass>>()
    val cartItems: LiveData<List<CartClass>> = _cartItems
    fun updateCartItems(newItems: List<CartClass>) {
        _cartItems.value = newItems
    }
}