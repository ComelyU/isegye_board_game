package com.example.presentation.adapter

import android.view.View
import androidx.databinding.BindingAdapter
import com.example.presentation.R

@BindingAdapter("turtleSelected", "turtleId")
fun setBackgroundBasedOnSelection(view: View, selectedTurtle: Int?, turtleId: Int) {
    view.setBackgroundColor(
        if (selectedTurtle == turtleId) view.context.getColor(R.color.selectedColor)
        else view.context.getColor(R.color.unselectedColor)
    )
}