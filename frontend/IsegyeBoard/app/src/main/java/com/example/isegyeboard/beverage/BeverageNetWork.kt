package com.example.isegyeboard.beverage

import com.example.isegyeboard.baseapi.BaseApi

class BeverageNetWork{
    private val client = BaseApi.getInstance().create(BeverageApi::class.java)

    suspend fun getMenuList() = client.getMenuList("1")

}