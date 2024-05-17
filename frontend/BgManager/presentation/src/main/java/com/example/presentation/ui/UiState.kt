package com.example.presentation.ui

data class UiState (
    val orders: List<OrderUiState>,
    val games: List<GameUiState>,
    val turtles: List<TurtleUiState>
)