package com.example.presentation.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.example.presentation.databinding.ItemlayoutTurtleBinding
import com.example.presentation.ui.TurtleUiState

class TurtleAdapter(private val turtleOnClickListener: TurtleOnClickListener)
    : ListAdapter<TurtleUiState, TurtleAdapter.TurtleViewHolder>(TurtleDiffCallback()) {

    class TurtleViewHolder(private val binding: ItemlayoutTurtleBinding, private val clickListener: TurtleOnClickListener
    ) : RecyclerView.ViewHolder(binding.root) {

        fun bind(item: TurtleUiState) {
            binding.turtleItem = item
            binding.turtleSelectBButton.setOnClickListener {
                clickListener.onTurtleClicked(item.turtleId)
            }
            binding.executePendingBindings()
//            println("터틀 어댑터 뷰홀더 들어옴 ${binding.turtleItem}")
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): TurtleViewHolder {
        val inflater = LayoutInflater.from(parent.context)
        val binding = ItemlayoutTurtleBinding.inflate(inflater, parent, false)
        return TurtleViewHolder(binding, turtleOnClickListener)
    }

    override fun onBindViewHolder(holder: TurtleViewHolder, position: Int) {
        val item = getItem(position)
//        println("터틀 바인드 뷰홀더 들어옴 $item")
        holder.bind(item)
    }

    private class TurtleDiffCallback : DiffUtil.ItemCallback<TurtleUiState>() {
        override fun areItemsTheSame(oldItem: TurtleUiState, newItem: TurtleUiState): Boolean {
            return oldItem.turtleId == newItem.turtleId
        }

        override fun areContentsTheSame(oldItem: TurtleUiState, newItem: TurtleUiState): Boolean {
            return oldItem == newItem
        }
    }

    interface TurtleOnClickListener {
        fun onTurtleClicked(turtleId: Int)
    }
}