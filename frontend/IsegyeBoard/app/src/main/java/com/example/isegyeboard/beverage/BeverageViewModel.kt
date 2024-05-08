package com.example.isegyeboard.beverage

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.isegyeboard.beverage.model.BeverageClass
import kotlinx.coroutines.launch

class CoffeeViewModel : ViewModel()  {
    private val beverageNetwork = BeverageNetWork()

    private val _menuList = MutableLiveData<List<BeverageClass>>()
    val coffeeMenuList: LiveData<List<BeverageClass>> = _menuList

    fun getCurrentMenuList() {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList()
                val coffeeMenu = result.filter { it.menuType == "C" }
                _menuList.value = coffeeMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}

class DrinkViewModel : ViewModel()  {
    private val beverageNetwork = BeverageNetWork()

    private val _menuList = MutableLiveData<List<BeverageClass>>()
    val drinkMenuList: LiveData<List<BeverageClass>> = _menuList

    fun getCurrentMenuList() {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList()
                val DrinkMenu = result.filter { it.menuType == "D" }
                _menuList.value = DrinkMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}

class SnackViewModel : ViewModel() {
    private val beverageNetwork = BeverageNetWork()

    private val _menuList = MutableLiveData<List<BeverageClass>>()
    val snackMenuList: LiveData<List<BeverageClass>> = _menuList

    fun getCurrentMenuList() {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList()
                val snackMenu = result.filter { it.menuType == "F" }
                _menuList.value = snackMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}
