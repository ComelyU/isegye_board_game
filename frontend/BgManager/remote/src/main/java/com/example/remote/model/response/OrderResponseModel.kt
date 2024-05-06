package com.example.remote.model.response

import com.google.gson.annotations.SerializedName

data class OrderResponseModel (
    @SerializedName("id") val orderId: Int = 0,
    @SerializedName("customerId") val customerId: Int = 0,
    @SerializedName("orderStatus") val orderStatus: Int = 0,
    @SerializedName("orderMenuDetail") val orderDetailData : List<OrderDetailResponseModel>
)