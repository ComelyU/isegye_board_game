package com.example.remote.retrofit

import com.example.remote.model.response.OrderResponseModel
import com.example.remote.model.response.TurtleBotResponseModel
import retrofit2.Response
import retrofit2.http.GET


interface ApiService {
    @GET("turtle")
    suspend fun getTurtleBot(
    ): Response<TurtleBotResponseModel>

    @GET("order-list")
    suspend fun getOrderList()
    : Response<List<OrderResponseModel>>
}