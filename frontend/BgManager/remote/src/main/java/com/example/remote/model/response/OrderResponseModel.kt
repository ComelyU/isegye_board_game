package com.example.remote.model.response

import com.google.gson.annotations.SerializedName

data class OrderResponseModel (
    @SerializedName("id") val id: Int = 0,
    @SerializedName("orderName") val orderName: String = "",
    @SerializedName("quantity") val quantity: Int = 0
)