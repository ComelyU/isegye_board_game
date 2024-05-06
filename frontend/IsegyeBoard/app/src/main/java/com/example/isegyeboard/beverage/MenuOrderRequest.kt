package com.example.isegyeboard.beverage

data class MenuOrderRequest(
    val CreateOrderMenuRequest: List<CreateOrderMenuRequest>
)

data class CreateOrderMenuRequest(
    val menuId: Int,
    val quantity: Int
)

