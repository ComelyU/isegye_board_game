package com.example.remote.model.response

import com.google.gson.annotations.SerializedName

data class BaseResponseModel (
    @SerializedName("result") val result: Boolean = false,
    @SerializedName("message") val message: String = "",
)