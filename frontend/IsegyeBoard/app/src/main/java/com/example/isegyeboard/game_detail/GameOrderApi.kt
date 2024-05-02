package com.example.isegyeboard.game_detail

import com.example.isegyeboard.baseapi.BasicResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.PUT
import retrofit2.http.Query

interface GameOrderApi {

    // 게임 배달 요청
    @GET("game-order?")
    fun orderGame(
        @Query("gameId") gameId: String,
        @Query("roomLogId") roomLogId: String
    ): Call<BasicResponse>

    // 게임 반납 요청
    @GET("game-order/return?")
    fun returnGame(
        @Query("roomLogId") roomLogId: String
    ): Call<BasicResponse>

    //테마 요청(게임도착시)
    @POST("room/theme")
    fun themeRequest(@Body requestBody: Map<String, String?>): Call<BasicResponse>

    //테마 on/off 여부
    @GET("room/theme")
    fun themeCheck(
        @Query("roomLogId") roomLogId: String,
    ): Call<Boolean>

    //테마 토글
    @PUT("room/theme")
    fun sendThemeToggle(@Body requestBody: Map<String, String?>): Call<BasicResponse>
}