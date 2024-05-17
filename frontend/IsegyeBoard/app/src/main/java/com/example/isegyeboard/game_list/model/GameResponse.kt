package com.example.isegyeboard.game_list.model

data class GameResponse(
    val id: Int,
    val themeType: String,
    val gameName:String,
    val gameDetail:	String,
    val minPlayer:Int,
    val maxPlayer:Int,
    val minPlaytime:Int,
    val maxPlaytime:Int,
    val gameDifficulty:Float,
    val gameImgUrl:String,
    val gameTagCategory: List<GameTagCategory>
)
