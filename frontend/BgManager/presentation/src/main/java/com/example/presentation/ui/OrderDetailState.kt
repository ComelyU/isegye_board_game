package com.example.presentation.ui

data class OrderDetailState(
    val orderDetailId: Int,
    val menuName: String,
    val quantity: Int,
    val totalPrice: Int
) {
    val quantityString: String
        get() = quantity.toString()

    val totalPriceString: String
        get() = totalPrice.toString()
}
