package com.example.remote.model.response

import com.google.gson.annotations.SerializedName

data class OrderDetailResponseModel(
    @SerializedName("id") val orderDetailId: Int,
    @SerializedName("menuName") val menuName: String,
    @SerializedName("quantity") val quantity: Int,
    @SerializedName("totalPrice") val totalPrice: Int
)
