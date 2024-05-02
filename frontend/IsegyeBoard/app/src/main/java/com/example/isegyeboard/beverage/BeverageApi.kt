package com.example.isegyeboard.beverage

import retrofit2.http.GET
import retrofit2.http.Query

interface BeverageApi {
    @GET("menu/list?")
    suspend fun getMenuList(
        @Query("storeId") storeId: String,
    ) : List<BeverageClass>
}