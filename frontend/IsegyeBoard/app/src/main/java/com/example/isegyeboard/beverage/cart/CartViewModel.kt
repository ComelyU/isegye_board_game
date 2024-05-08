package com.example.isegyeboard.beverage.cart

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.isegyeboard.beverage.cart.CartClass

class CartViewModel : ViewModel() {
    private val _cartItems = MutableLiveData<List<CartClass>>()
    val cartItems: LiveData<List<CartClass>> = _cartItems

    fun updateCartItems(newItems: List<CartClass>) {
        _cartItems.value = newItems
    }
}
