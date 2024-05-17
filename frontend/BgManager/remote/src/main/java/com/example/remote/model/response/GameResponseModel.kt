package com.example.remote.model.response

import com.google.gson.annotations.SerializedName

data class GameResponseModel(
    @SerializedName("id") val gameOrderId: Int,
    @SerializedName("gameName") val gameName: String,
    @SerializedName("customerRoomNumber") val roomNumber: Int,
    @SerializedName("customerId") val customerId: Int,
    @SerializedName("stockLocation") val stockLocation: String,
    @SerializedName("orderType") val orderType: Int,
    @SerializedName("orderStatus") val orderStatus: Int,
)
