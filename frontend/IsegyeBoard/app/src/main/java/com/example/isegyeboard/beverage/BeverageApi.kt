package com.example.isegyeboard.beverage

import com.example.isegyeboard.beverage.model.BeverageClass
import com.example.isegyeboard.beverage.model.CreateOrderMenuRequest
import com.example.isegyeboard.room_history.OrderMenuResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path

interface BeverageApi {
    @GET("menu/{storeId}")
    suspend fun getMenuList(
        @Path("storeId") storeId: String,
    ) : List<BeverageClass>

    @POST("menu/order/{customerId}")
    fun menuOrder(
        @Path("customerId") customerId: String,
        @Body requestBody: List<CreateOrderMenuRequest>
    ): Call<OrderMenuResponse>
}