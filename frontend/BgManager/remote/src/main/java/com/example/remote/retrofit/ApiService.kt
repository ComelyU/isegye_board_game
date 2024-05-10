package com.example.remote.retrofit

import com.example.remote.model.request.DeliverRequestModel
import com.example.remote.model.response.DeliverResponseModel
import com.example.remote.model.response.OrderGameList
import com.example.remote.model.response.OrderResponseModel
import com.example.remote.model.response.TurtleBotResponseModel
import retrofit2.Response
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path


interface ApiService {
    @GET("turtle/{storeId}/list")
    suspend fun getTurtleBot(
        @Path("storeId") storeId: String
    ): Response<List<TurtleBotResponseModel>>

    @GET("menu/order/store/{storeId}")
    suspend fun getOrderList(
        @Path("storeId") storeId: String
    )
    : Response<List<OrderResponseModel>>

    @GET("game/order/stores/{storeId}")
    suspend fun getGameList(
        @Path("storeId") storeId: String
    )
    : Response<OrderGameList>

    @POST("turtle/order/{turtleId}")
    suspend fun startDelivery(
        @Path("turtleId") turtleId : Int,
        @Body requestBody: DeliverRequestModel
    ) : Response<DeliverResponseModel>
}