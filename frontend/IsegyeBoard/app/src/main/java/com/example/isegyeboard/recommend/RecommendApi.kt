package com.example.isegyeboard.recommend

import com.example.isegyeboard.game_list.model.GameResponse
import retrofit2.Call
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path
import retrofit2.http.Query

interface RecommendApi {
    @GET("customer/filter")
    fun sendRecommendData(
        @Query("theme") theme: String,
        @Query("difficulty") difficulty: String,
        @Query("tag") tag: String,
        @Query("time") time: String
    ): Call<RecommendGameResponse>

    @GET("game/{gameId}")
    fun recommendTest(
        @Path("gameId") gameId: String
    ): Call<GameResponse>
}