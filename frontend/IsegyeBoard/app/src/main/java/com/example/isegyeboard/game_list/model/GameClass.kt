package com.example.isegyeboard.game_list.model

data class GameClass(
    val id: Int,
    val isAvailable: Int,
    val stockLocation: String,
    val game: GameResponse, // 테마
)
