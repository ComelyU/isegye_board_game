package com.example.bgmanager

import retrofit2.http.GET
import retrofit2.http.Query

interface OrderListApi {
    @GET("menu-order/orderList?")
    suspend fun getOrderList(
        @Query("storeId") storeId: String,
    ) : List<OrderClass>
}