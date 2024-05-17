package com.example.data.model

data class OrderDetailData(
    val orderDetailId: Int,
    val menuName: String,
    val quantity: Int,
    val totalPrice: Int
)