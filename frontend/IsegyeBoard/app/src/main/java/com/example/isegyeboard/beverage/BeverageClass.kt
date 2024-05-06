package com.example.isegyeboard.beverage

data class BeverageClass(
    val id: Int,
    val menuName: String,
    val menuType: String,
    val menuPrice: Int,
    val menuImgUrl: String,
    val isAvailable: Int?,
)
