package com.example.isegyeboard.beverage.cart

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.isegyeboard.beverage.cart.CartClass

class CartViewModel : ViewModel() {
    private val _cartItems = MutableLiveData<List<CartClass>>()
    val cartItems: LiveData<List<CartClass>> = _cartItems

    fun addCartItem(cartItem: CartClass) {
        val currentList = _cartItems.value ?: emptyList()
        val newList = currentList.toMutableList()
        newList.add(cartItem)
        _cartItems.value = newList
    }

    fun removeCartItem(cartItem: CartClass) {
        val currentList = _cartItems.value ?: return
        val newList = currentList.toMutableList()
        newList.remove(cartItem)
        _cartItems.value = newList
    }
}