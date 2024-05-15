package com.example.isegyeboard.recommend

import com.example.isegyeboard.game_list.model.GameResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path

interface RecommendApi {
    @POST("recommend")
    fun sendRecommendData(@Body requestBody: Map<String, String?>): Call<GameResponse>

    @GET("game/{gameId}")
    fun recommendTest(
        @Path("gameId") gameId: String
    ): Call<GameResponse>
}