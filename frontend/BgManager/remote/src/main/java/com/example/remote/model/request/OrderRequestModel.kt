package com.example.remote.model.request

import com.google.gson.annotations.SerializedName

data class OrderRequestModel(
    @SerializedName("storeId") val id: Int = 1,
)
