package com.example.isegyeboard.room

import com.example.isegyeboard.baseapi.BasicResponse
import com.example.isegyeboard.room_history.OrderMenuResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PATCH
import retrofit2.http.POST
import retrofit2.http.PUT
import retrofit2.http.Path

interface RoomApi {
    @POST("customer/{roomId}")
    fun sendRoomInfo(
        @Path("roomId") roomId: String,
        @Body requestBody: Map<String, Int>
    ): Call<RoomStartResponse>

    @PATCH("customer/{customerId}")
    fun deleteCustomerInfo(
        @Path("customerId") customerId: String
    ): Call<RoomOverResponse>

    @GET("menu/order/{customerId}")
    suspend fun getHistoryList(
        @Path("customerId") customerId: String,
    ) : List<OrderMenuResponse>

}