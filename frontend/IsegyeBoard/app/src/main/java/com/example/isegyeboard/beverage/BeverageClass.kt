package com.example.isegyeboard.beverage

data class BeverageClass(
    val id: Int,
    val name: String,
    val type: Int,
    val price: Int,
//    val imageURL: String,
    val isAvailable: Boolean?,
    val storeId: Int,
)
