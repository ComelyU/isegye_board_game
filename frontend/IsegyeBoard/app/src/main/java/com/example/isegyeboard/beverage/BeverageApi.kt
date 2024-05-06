package com.example.isegyeboard.beverage

import com.example.isegyeboard.baseapi.BasicResponse
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

    @POST("menu/order{customerId}")
    fun menuOrder(
        @Path("customerId") customerId: String,
        @Body requestBody: MenuOrderRequest
    ): Call<BasicResponse>
}