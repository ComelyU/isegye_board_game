package com.example.remote.model.response

import com.google.gson.annotations.SerializedName

data class TurtleBotResponseModel(
    @SerializedName("id") val id: Int = 0,
    @SerializedName("storeName") val storeId: Int = 0
)
