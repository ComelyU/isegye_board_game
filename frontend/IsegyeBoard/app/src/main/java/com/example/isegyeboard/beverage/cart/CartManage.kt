package com.example.isegyeboard.beverage.cart

class CartManage private constructor() {
    private val items: MutableList<CartClass> = mutableListOf()

    companion object {
        @Volatile
        private var instance: CartManage? = null

        fun getInstance(): CartManage {
            return instance ?: synchronized(this) {
                instance ?: CartManage().also { instance = it }
            }
        }
    }

    fun getItems(): List<CartClass> {
        return items
    }

    fun addItem(item: CartClass) {
        val existingItem = items.find { it.name == item.name }
        if (existingItem != null) {
            // 이미 장바구니에 있는 아이템인 경우 수량을 증가시킵니다.
            existingItem.quantity++
        } else {
            // 장바구니에 없는 아이템인 경우 새로 추가합니다.
            items.add(item)
        }
    }

    fun removeItem(item: CartClass) {
        items.remove(item)
    }

    fun clearCart() {
        items.clear()
    }

}