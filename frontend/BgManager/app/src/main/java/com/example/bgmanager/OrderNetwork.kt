package com.example.bgmanager

class OrderNetwork {
    private val client = BaseApi.getInstance().create(OrderListApi::class.java)

    suspend fun getOrderList() = client.getOrderList("1");
}