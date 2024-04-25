package com.example.bgmanager

data class OrderClass(
    val id: Int,
    val menuName: String,
    val quantity: Int,
    val createdDttm: String,
    val roomLogId: String,
    val isDelivered: Boolean
)
