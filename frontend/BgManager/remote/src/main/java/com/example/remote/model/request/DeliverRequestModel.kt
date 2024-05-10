package com.example.remote.model.request

import com.google.gson.annotations.SerializedName

data class DeliverRequestModel(
    @SerializedName("orderMenuId") val orderMenuId: Int?,
    @SerializedName("orderGameId") val orderGameId: Int?,
    @SerializedName("returnGameId") val returnGameId: Int?,
)
