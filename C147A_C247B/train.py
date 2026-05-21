"""
Training file for the models we implemented 
"""

from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.utils
from torch.utils.data import DataLoader
from einops import rearrange
import wandb

from model import BigramLanguageModel, MiniGPT
from dataset import TinyStoriesDataset
from config import BigramConfig, MiniGPTConfig

def solver(model_name):
    # Initialize the model
    if model_name == "bigram":
        config = BigramConfig
        model = BigramLanguageModel(config)
    elif model_name == "minigpt":
        config = MiniGPTConfig
        model = MiniGPT(config)
    else:
        raise ValueError("Invalid model name")
    
    # Load the dataset
    train_dataset = TinyStoriesDataset(
        config.path_to_data,
        mode="train",
        context_length=config.context_length,
    )
    eval_dataset = TinyStoriesDataset(
        config.path_to_data, mode="test", context_length=config.context_length
    )

    # Create the dataloaders
    train_dataloader = DataLoader(
        train_dataset, batch_size=config.batch_size, pin_memory=True
    )
    eval_dataloader = DataLoader(
        eval_dataset, batch_size=config.batch_size, pin_memory=True
    )

    # Set the device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def count_parameters(model):
        return sum(p.numel() for p in model.parameters() if p.requires_grad)
    print("number of trainable parameters: %.2fM" % (count_parameters(model) / 1e6,))

    if config.to_log:
        wandb.init(project="dl2_proj3")

    if not Path.exists(config.save_path):
        Path.mkdir(config.save_path, parents=True, exist_ok=True)

    # Define the loss function
    loss_fn = nn.CrossEntropyLoss()

    # Define the optimizer
    optimizer = torch.optim.AdamW(model.parameters(), lr=config.learning_rate)

    if config.scheduler:
        scheduler = torch.optim.lr_scheduler.CosineAnnealingWarmRestarts(
            optimizer, T_0=2000, T_mult=2
        )

    model.train()
    model.to(device)

    for i, (context, target) in enumerate(train_dataloader):
        context, target = context.to(device), target.to(device)
        
        # Forward pass
        logits = model(context)
        
        # Reshape to match the cross-entropy expectations (B*T, C)
        B, T, C = logits.shape
        loss = loss_fn(logits.view(B * T, C), target.view(B * T))
        
        # Backward pass & Optimize
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        optimizer.step()
        
        train_loss = loss.item()


        if config.scheduler:
            scheduler.step()

        del context, target # Clear memory

        if i % config.log_interval == 0:
            model.eval()

            # Run evaluation loop on a few batches so it doesn't take forever
            eval_steps = 10
            total_eval_loss = 0.0
            
            with torch.no_grad():
                for e_idx, (e_ctx, e_tgt) in enumerate(eval_dataloader):
                    if e_idx >= eval_steps:
                        break
                    
                    e_ctx, e_tgt = e_ctx.to(device), e_tgt.to(device)
                    e_logits = model(e_ctx)
                    e_B, e_T, e_C = e_logits.shape
                    
                    e_loss = loss_fn(e_logits.view(e_B * e_T, e_C), e_tgt.view(e_B * e_T))
                    total_eval_loss += e_loss.item()
                    
            eval_loss = total_eval_loss / eval_steps
            
            print(
                f"Iteration {i}, Train Loss: {train_loss:.4f}",
                f"Eval Loss: {eval_loss:.4f}",
            )

            if config.to_log:
                wandb.log(
                    {
                        "Train Loss": train_loss,
                        "Eval Loss": eval_loss,
                    }
                )

            model.train()

        # Save the model every config.save_iterations
        if i % config.save_iterations == 0:
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "optimizer_state_dict": optimizer.state_dict(),
                    "train_loss": train_loss,
                    "eval_loss": eval_loss,
                    "iteration": i,
                },
                config.save_path / f"mini_model_checkpoint_{i}.pt",
            )

        if i > config.max_iter:
            break