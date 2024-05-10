package com.example.presentation.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.example.presentation.databinding.ItemlayoutGameBinding
import com.example.presentation.ui.GameUiState

class GameAdapter(private val gameOnClickListener: GameOnClickListener)
    : ListAdapter<GameUiState, GameAdapter.GameViewHolder>(GameDiffCallback()) {

    class GameViewHolder(private val binding: ItemlayoutGameBinding, private val clickListener: GameAdapter.GameOnClickListener
        ) : RecyclerView.ViewHolder(binding.root) {

        fun bind(item: GameUiState) {
            binding.gameItem = item
            binding.gameCartButton.setOnClickListener{
                clickListener.onGameClicked(item.gameOrderId, item.orderType, item.roomNumber)
            }
            binding.executePendingBindings()
//            println("게임 어댑터 뷰홀더 들어옴 ${binding.gameItem}")
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): GameViewHolder {
        val inflater = LayoutInflater.from(parent.context)
        val binding = ItemlayoutGameBinding.inflate(inflater, parent, false)
        return GameViewHolder(binding, gameOnClickListener)
    }

    override fun onBindViewHolder(holder: GameViewHolder, position: Int) {
//        val item = itemList[position]
        val item = getItem(position)
//        println("게임 바인드 뷰홀더 들어옴 $item")
        holder.bind(item)
    }

    private class GameDiffCallback : DiffUtil.ItemCallback<GameUiState>() {
        override fun areItemsTheSame(oldItem: GameUiState, newItem: GameUiState): Boolean {
            return oldItem.gameOrderId == newItem.gameOrderId
        }

        override fun areContentsTheSame(oldItem: GameUiState, newItem: GameUiState): Boolean {
            return oldItem == newItem
        }
    }

    interface GameOnClickListener {
        fun onGameClicked(gameOrderId: Int, orderType: Int, roomNumber: Int)
    }
}