package com.example.isegyeboard.beverage

import retrofit2.http.GET
import retrofit2.http.Path

interface BeverageApi {
    @GET("menu/{storeId}")
    suspend fun getMenuList(
        @Path("storeId") storeId: String,
    ) : List<BeverageClass>
}