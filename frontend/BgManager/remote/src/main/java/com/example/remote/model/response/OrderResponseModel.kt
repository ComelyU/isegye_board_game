package com.example.remote.model.response

import com.google.gson.annotations.SerializedName

data class OrderResponseModel (
    @SerializedName("id") val orderId: Int,
    @SerializedName("customerId") val customerId: Int,
    @SerializedName("orderStatus") val orderStatus: Int,
    @SerializedName("customerRoomNumber") val roomNumber: Int,
    @SerializedName("orderMenuDetail") val orderDetailData : List<OrderDetailResponseModel>
)