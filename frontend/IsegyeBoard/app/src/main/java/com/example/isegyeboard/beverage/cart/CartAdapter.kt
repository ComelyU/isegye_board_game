package com.example.isegyeboard.beverage.cart

import android.content.Context
import android.graphics.Color
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.isegyeboard.R
import com.example.isegyeboard.beverage.cart.CartClass

class CartAdapter(
    private val context: Context,
    private var cartItems: List<CartClass>,
    private val itemClickListener: OnItemClickListener
) : RecyclerView.Adapter<CartAdapter.CartViewHolder>() {

    interface OnItemClickListener {
        fun onItemRemoved(cartItem: CartClass)
    }

    inner class CartViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView) {
        val nameTextView: TextView = itemView.findViewById(R.id.cartName)
        val quantityTextView: TextView = itemView.findViewById(R.id.cartQuantity)
        val minusTextView: TextView = itemView.findViewById(R.id.minusButton)

        init {
            minusTextView.setOnClickListener {
                val position = adapterPosition
                if (position != RecyclerView.NO_POSITION) {
                    val removedItem = cartItems[position]
                    itemClickListener.onItemRemoved(removedItem)
                }
            }
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): CartViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.layout_cart_item, parent, false)
        return CartViewHolder(view)
    }

    override fun onBindViewHolder(holder: CartViewHolder, position: Int) {
        val cartItem = cartItems[position]

        holder.nameTextView.text = cartItem.name
        holder.quantityTextView.text = "${cartItem.price} 원"
    }

    override fun getItemCount(): Int {
        return cartItems.size
    }

    fun updateData(newCartItems: List<CartClass>) {
        cartItems = newCartItems
        notifyDataSetChanged()
    }
}