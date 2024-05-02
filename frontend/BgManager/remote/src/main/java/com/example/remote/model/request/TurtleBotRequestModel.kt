package com.example.remote.model.request

import com.google.gson.annotations.SerializedName

data class TurtleBotRequestModel (
    @SerializedName("storeId") val id: Int = 0,
)