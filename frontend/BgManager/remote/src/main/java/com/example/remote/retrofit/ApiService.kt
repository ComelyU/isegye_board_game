package com.example.remote.retrofit

import com.example.remote.model.response.OrderResponseModel
import com.example.remote.model.response.TurtleBotResponseModel
import retrofit2.Response
import retrofit2.http.GET
import retrofit2.http.Path


interface ApiService {
    @GET("turtle")
    suspend fun getTurtleBot(
    ): Response<TurtleBotResponseModel>

    @GET("menu/order/store/{storeId}")
    suspend fun getOrderList(
        @Path("storeId") storeId: String
    )
    : Response<List<OrderResponseModel>>
}