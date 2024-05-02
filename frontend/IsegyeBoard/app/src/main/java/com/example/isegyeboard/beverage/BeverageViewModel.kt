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

    private val _cartItems = MutableLiveData<List<CartClass>>()

    val cartItems: LiveData<List<CartClass>> = _cartItems

    fun getMenuList(storeId: String) {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList(storeId)
                val allMenu = result
                _menuList.value = allMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }


    fun updateCartItems(newItems: List<CartClass>) {
        _cartItems.value = newItems
    }

    //    private val _coffeeList = MutableLiveData<List<BeverageClass>>()
//    private val _drinkList = MutableLiveData<List<BeverageClass>>()
//    private val _snackList = MutableLiveData<List<BeverageClass>>()


    //
//    val coffeeMenuList: LiveData<List<BeverageClass>> = _coffeeList
//    val drinkMenuList: LiveData<List<BeverageClass>> = _drinkList
//    val snackMenuList: LiveData<List<BeverageClass>> = _snackList

    //    fun getCoffeeList(storeId: String) {
//        viewModelScope.launch {
//            try {
//                val result = beverageNetwork.getMenuList(storeId)
//                val coffeeMenu = result.filter { it.type == 0 }
//                _coffeeList.value = coffeeMenu
//            } catch (e: Exception) {
//                // 에러 처리
//                println(e)
//            }
//        }
//    }
//
//    fun getDrinkList(storeId: String) {
//        viewModelScope.launch {
//            try {
//                val result = beverageNetwork.getMenuList(storeId)
//                val drinkMenu = result.filter { it.type == 1 }
//                _drinkList.value = drinkMenu
//            } catch (e: Exception) {
//                // 에러 처리
//                println(e)
//            }
//        }
//    }
//
//    fun getSnackList(storeId: String) {
//        viewModelScope.launch {
//            try {
//                val result = beverageNetwork.getMenuList(storeId)
//                val snackMenu = result.filter { it.type == 2 }
//                _snackList.value = snackMenu
//            } catch (e: Exception) {
//                // 에러 처리
//                println(e)
//            }
//        }
//    }
}

//class DrinkViewModel : ViewModel()  {
//    private val beverageNetwork = BaseApi.getInstance().create(BeverageApi::class.java)
//
//    private val _menuList = MutableLiveData<List<BeverageClass>>()
//    val drinkMenuList: LiveData<List<BeverageClass>> = _menuList
//
//    fun getCurrentMenuList(storeId: String) {
//        viewModelScope.launch {
//            try {
//                val result = beverageNetwork.getMenuList(storeId)
//                val drinkMenu = result.filter { it.type == 1 }
//                _menuList.value = drinkMenu
//            } catch (e: Exception) {
//                // 에러 처리
//                println(e)
//            }
//        }
//    }
//}
//
//class SnackViewModel : ViewModel()  {
//    private val beverageNetwork = BaseApi.getInstance().create(BeverageApi::class.java)
//
//    private val _menuList = MutableLiveData<List<BeverageClass>>()
//    val snackMenuList: LiveData<List<BeverageClass>> = _menuList
//
//    fun getCurrentMenuList(storeId: String) {
//        viewModelScope.launch {
//            try {
//                val result = beverageNetwork.getMenuList(storeId)
//                val snackMenu = result.filter { it.type == 2 }
//                _menuList.value = snackMenu
//            } catch (e: Exception) {
//                // 에러 처리
//                println(e)
//            }
//        }
//    }
//}