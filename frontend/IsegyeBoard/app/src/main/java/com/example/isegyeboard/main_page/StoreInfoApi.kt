package com.example.isegyeboard.main_page

import retrofit2.Call
import retrofit2.http.GET
import retrofit2.http.Path

interface StoreInfoApi {
    @GET("stores/{storeId}")
    fun getStoreInfo(
        @Path("storeId") storeId: String
    ): Call<StoreClass>
}