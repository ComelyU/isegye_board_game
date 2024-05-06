package com.example.isegyeboard.beverage.cart

data class CartClass(
    val name: String,
    val price: Int,
    var quantity: Int = 1 // 수량 필드 추가, 기본값은 1
)